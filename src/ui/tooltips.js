export function installTooltips(document, window) {
    const infoPopup = document.createElement('div');
    infoPopup.className = 'info-popup';
    infoPopup.setAttribute('role', 'dialog');
    infoPopup.setAttribute('aria-hidden', 'true');
    document.body.appendChild(infoPopup);
    let activeInfoButton = null;

    function hideInfoPopup() {
        if (!infoPopup.classList.contains('visible')) return;
        infoPopup.classList.remove('visible');
        infoPopup.setAttribute('aria-hidden', 'true');
        infoPopup.textContent = '';
        activeInfoButton = null;
    }

    function showInfoPopup(button) {
        const message = button?.dataset?.info;
        if (!message) return;
        infoPopup.textContent = message;
        infoPopup.classList.add('visible');
        infoPopup.setAttribute('aria-hidden', 'false');
        const rect = button.getBoundingClientRect();
        const { innerWidth } = window;
        const scrollX = window.scrollX || window.pageXOffset;
        const scrollY = window.scrollY || window.pageYOffset;
        const popupRect = infoPopup.getBoundingClientRect();
        const top = scrollY + rect.bottom + 8;
        let left = scrollX + rect.left + rect.width / 2 - popupRect.width / 2;
        const minLeft = scrollX + 8;
        const maxLeft = scrollX + innerWidth - popupRect.width - 8;
        if (left < minLeft) left = minLeft;
        if (left > maxLeft) left = Math.max(minLeft, maxLeft);
        infoPopup.style.top = `${top}px`;
        infoPopup.style.left = `${left}px`;
        activeInfoButton = button;
    }

    document.addEventListener('click', (event) => {
        const button = event.target.closest('.info-button');
        if (button) {
            event.preventDefault();
            event.stopPropagation();
            if (activeInfoButton === button && infoPopup.classList.contains('visible')) {
                hideInfoPopup();
            } else {
                showInfoPopup(button);
            }
            return;
        }
        if (!infoPopup.contains(event.target)) {
            hideInfoPopup();
        }
    });

    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape') {
            hideInfoPopup();
        }
    });

    window.addEventListener('scroll', hideInfoPopup, true);
    window.addEventListener('resize', hideInfoPopup);


}
